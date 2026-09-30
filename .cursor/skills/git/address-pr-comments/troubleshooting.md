# Troubleshooting

## Pending Review 422 Error

If replying to a review comment returns a **422 error** about pending reviews, submit the pending review first:

```bash
REVIEW_ID=$(gh api "repos/$OWNER_REPO/pulls/$PR_NUMBER/reviews" \
  --jq '.[] | select(.state == "PENDING") | .id' | head -1)

gh api "repos/$OWNER_REPO/pulls/$PR_NUMBER/reviews/$REVIEW_ID/events" \
  -X POST -f event="COMMENT" -f body=""
```

Then retry the reply.
